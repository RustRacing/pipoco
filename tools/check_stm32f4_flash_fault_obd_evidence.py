#!/usr/bin/env python3
"""Validate STM32F4 FlashKV OBD 0x09/0xE2 fault evidence."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

SCHEMA = "stm32f4-flash-fault-obd-evidence-v1"
DEFAULT_EVIDENCE = Path("target/hardware-evidence/latest/stm32f4-flash-fault-obd.json")

EXPECTED_DISCOVERY = {
    "service": 0x49,
    "parameter_id": 0xE0,
    "payload": [0xC0, 0x00, 0x00, 0x00],
}

EXPECTED_FAULT = {
    "service": 0x49,
    "parameter_id": 0xE2,
    "sequence_index": 0,
    "segment_count": 1,
    "total_payload_len": 6,
    "segment_len": 6,
}

PHASE_CODES = {
    "preflight": 1,
    "erase": 2,
    "program": 3,
}

STM32F405_FLASH_ERROR_MASK = (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7)
SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")
HEX_RE = re.compile(r"^(?:[0-9a-fA-F]{2})(?:[ :_-]?[0-9a-fA-F]{2})*$")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate hardware-captured OBD 0x09/0xE2 FlashKV fault evidence."
    )
    parser.add_argument(
        "evidence",
        nargs="?",
        default=str(DEFAULT_EVIDENCE),
        help=f"evidence JSON path (default: {DEFAULT_EVIDENCE})",
    )
    return parser.parse_args(argv)


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError("evidence must be a JSON object")
    return data


def byte_list(value: Any, label: str, errors: list[str]) -> list[int]:
    if not isinstance(value, list):
        errors.append(f"{label} must be an array of bytes")
        return []
    out = []
    for idx, item in enumerate(value):
        if not isinstance(item, int) or item < 0 or item > 0xFF:
            errors.append(f"{label}[{idx}] must be byte 0..255")
        else:
            out.append(item)
    return out


def int_field(data: dict[str, Any], key: str, label: str, errors: list[str]) -> int | None:
    value = data.get(key)
    if not isinstance(value, int):
        errors.append(f"{label}.{key} must be integer")
        return None
    return value


def parse_hex_frame(value: Any, label: str, errors: list[str]) -> list[int]:
    if not isinstance(value, str) or not HEX_RE.fullmatch(value.strip()):
        errors.append(f"{label} must be non-empty hex bytes")
        return []
    compact = re.sub(r"[^0-9a-fA-F]", "", value)
    return [int(compact[idx : idx + 2], 16) for idx in range(0, len(compact), 2)]


def contains_subsequence(frame: list[int], expected: list[int]) -> bool:
    if not expected or len(expected) > len(frame):
        return False
    return any(frame[idx : idx + len(expected)] == expected for idx in range(len(frame) - len(expected) + 1))


def validate_transcript(data: dict[str, Any], errors: list[str]) -> None:
    transcript = data.get("transcript")
    if not isinstance(transcript, list) or len(transcript) < 4:
        errors.append("transcript must contain request/response frames for 0xE0 and 0xE2")
        return
    expected = [
        ("request", 0x09, 0xE0),
        ("response", 0x49, 0xE0),
        ("request", 0x09, 0xE2),
        ("response", 0x49, 0xE2),
    ]
    for idx, (direction, service, parameter_id) in enumerate(expected):
        entry = transcript[idx]
        if not isinstance(entry, dict):
            errors.append(f"transcript[{idx}] must be object")
            continue
        if entry.get("direction") != direction:
            errors.append(f"transcript[{idx}].direction must be {direction}")
        if entry.get("service") != service:
            errors.append(f"transcript[{idx}].service must be 0x{service:02X}")
        if entry.get("parameter_id") != parameter_id:
            errors.append(f"transcript[{idx}].parameter_id must be 0x{parameter_id:02X}")
        frame = parse_hex_frame(entry.get("raw_frame"), f"transcript[{idx}].raw_frame", errors)
        if not contains_subsequence(frame, [service, parameter_id]):
            errors.append(
                f"transcript[{idx}].raw_frame must contain service 0x{service:02X} and parameter 0x{parameter_id:02X}"
            )

    if errors:
        return
    discovery_payload = data["discovery_response"]["payload"]
    discovery_response_frame = parse_hex_frame(
        transcript[1].get("raw_frame"), "transcript[1].raw_frame", errors
    )
    if not contains_subsequence(discovery_response_frame, [0x49, 0xE0, *discovery_payload]):
        errors.append("transcript[1].raw_frame must contain decoded discovery response bytes")

    fault_segment = data["fault_response"]["segment"]
    fault_response_frame = parse_hex_frame(
        transcript[3].get("raw_frame"), "transcript[3].raw_frame", errors
    )
    expected_fault = [
        0x49,
        0xE2,
        data["fault_response"]["sequence_index"],
        data["fault_response"]["segment_count"],
        data["fault_response"]["total_payload_len"],
        data["fault_response"]["segment_len"],
        *fault_segment,
    ]
    if not contains_subsequence(fault_response_frame, expected_fault):
        errors.append("transcript[3].raw_frame must contain decoded fault response bytes")


def validate_discovery(data: dict[str, Any], errors: list[str]) -> None:
    discovery = data.get("discovery_response")
    if not isinstance(discovery, dict):
        errors.append("discovery_response must be object")
        return
    for key, expected in EXPECTED_DISCOVERY.items():
        if key == "payload":
            payload = byte_list(discovery.get(key), "discovery_response.payload", errors)
            if payload != expected:
                errors.append(
                    "discovery_response.payload must be exactly [0xC0,0,0,0]"
                )
        elif discovery.get(key) != expected:
            errors.append(f"discovery_response.{key} must be 0x{expected:02X}")


def validate_fault_segment(data: dict[str, Any], errors: list[str]) -> None:
    fault = data.get("fault_response")
    if not isinstance(fault, dict):
        errors.append("fault_response must be object")
        return
    for key, expected in EXPECTED_FAULT.items():
        if fault.get(key) != expected:
            errors.append(f"fault_response.{key} must be {expected}")

    segment = byte_list(fault.get("segment"), "fault_response.segment", errors)
    if len(segment) != 6:
        errors.append("fault_response.segment must contain exactly 6 bytes")
        return

    present = (segment[0] & 0x80) != 0
    if not present:
        errors.append("fault_response.segment[0] must set present bit 7")
    if segment[0] & 0x7F:
        errors.append("fault_response.segment[0] reserved bits must be zero")

    expected_phase = data.get("expected_phase")
    if expected_phase not in PHASE_CODES:
        errors.append("expected_phase must be one of preflight/erase/program")
    elif segment[1] != PHASE_CODES[expected_phase]:
        errors.append(
            f"fault_response phase {segment[1]} does not match expected_phase {expected_phase}"
        )

    observed_sr = int.from_bytes(bytes(segment[2:6]), "big")
    expected_sr = data.get("expected_sr_bits")
    if not isinstance(expected_sr, int):
        errors.append("expected_sr_bits must be integer")
    elif observed_sr != expected_sr:
        errors.append(
            f"fault_response sr_bits 0x{observed_sr:08X} != expected 0x{expected_sr:08X}"
        )
    if observed_sr & STM32F405_FLASH_ERROR_MASK == 0:
        errors.append("fault_response sr_bits must include an STM32F405 FLASH error bit")


def validate(data: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if data.get("schema") != SCHEMA:
        errors.append(f"schema must be {SCHEMA}")
    for key in [
        "captured_at_utc",
        "firmware_build_id",
        "board_id",
        "probe_id",
        "capture_tool",
        "can_interface",
        "operator_notes",
        "fault_injection_method",
    ]:
        if not isinstance(data.get(key), str) or not data.get(key).strip():
            errors.append(f"{key} must be non-empty string")
    firmware_sha256 = data.get("firmware_sha256")
    if not isinstance(firmware_sha256, str) or not SHA256_RE.fullmatch(firmware_sha256):
        errors.append("firmware_sha256 must be 64 hex characters")
    bitrate = data.get("can_bitrate")
    if not isinstance(bitrate, int) or bitrate <= 0:
        errors.append("can_bitrate must be positive integer")
    validate_discovery(data, errors)
    validate_fault_segment(data, errors)
    if not errors:
        validate_transcript(data, errors)
    return errors


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        data = read_json(Path(args.evidence))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"FAIL: {exc}")
        return 1
    errors = validate(data)
    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("PASS: STM32F4 FlashKV OBD fault evidence matches 0x09/0xE2 contract")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
