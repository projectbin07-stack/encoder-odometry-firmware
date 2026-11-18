"""Shared protocol test vectors (``test/vectors/protocol_vectors.json``).

The JSON file is read by both test suites: the C++ Unity test in
``test/test_protocol_vectors`` and the pytest suite. Each side encodes the
listed fields and must produce the listed bytes, and decodes the bytes and
must recover the fields. The CRC and COBS entries are independent of this
code: the CRC check value is the published one for CRC-16/CCITT-FALSE and the
COBS cases are the worked examples from the COBS literature.

Field values are chosen to be exactly representable in float32 so both sides
can compare with ``==``.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .protocol import (
    Diagnostics,
    Message,
    Odometry,
    ParamGet,
    ParamSet,
    ParamValue,
    SetPose,
    VelocityCommand,
    WheelState,
    cobs_encode,
    crc16_ccitt,
    encode_frame,
    encode_packet,
    message_to_dict,
)

VECTORS_RELATIVE_PATH = Path("test") / "vectors" / "protocol_vectors.json"

_MESSAGES: list[tuple[str, Message, int]] = [
    ("velocity forward", VelocityCommand(0.5, 0.0, 0.0), 0),
    ("velocity strafe and turn", VelocityCommand(-0.25, 0.125, 1.5), 7),
    (
        "wheel state",
        WheelState(
            t_us=123456,
            count=(1000, -1000, 2147483647, -2147483648),
            velocity=(10.5, -10.5, 0.0, 3.25),
            target=(10.0, -10.0, 0.0, 3.0),
            duty=(5000, -5000, 0, 10000),
        ),
        200,
    ),
    ("odometry", Odometry(4294967295, 1.5, -2.25, 3.0, 0.25, -0.125, 0.5), 255),
    (
        "diagnostics",
        Diagnostics(
            t_us=1000000,
            illegal_transitions=(0, 1, 2, 65536),
            loop_overruns=3,
            watchdog_trips=4,
            max_loop_us=850,
            rx_crc_errors=5,
            rx_frame_errors=6,
            flags=0x05,
        ),
        1,
    ),
    ("param get", ParamGet(1), 2),
    ("param set", ParamSet(9, 12.5), 3),
    ("param value", ParamValue(9, 0, 12.5), 4),
    ("param value out of range", ParamValue(14, 2, 45.0), 5),
    ("set pose zero", SetPose(0.0, 0.0, 0.0), 6),
]


def _cobs_cases() -> list[tuple[str, bytes]]:
    run_254 = bytes(range(1, 255))
    return [
        ("empty", b""),
        ("single zero", b"\x00"),
        ("two zeros", b"\x00\x00"),
        ("zero in middle", b"\x11\x22\x00\x33"),
        ("no zeros", b"\x11\x22\x33\x44"),
        ("trailing zero", b"\x11\x00\x00\x00"),
        ("254 non-zero bytes", run_254),
        ("zero then 254 non-zero", b"\x00" + run_254),
        ("255 non-zero bytes", run_254 + b"\xff"),
    ]


def build_vectors() -> dict[str, Any]:
    crc_cases: list[dict[str, Any]] = [
        {"name": "check value", "ascii": "123456789", "crc": 0x29B1},
        {"name": "empty", "ascii": "", "crc": 0xFFFF},
        {"name": "single A", "ascii": "A", "crc": 0xB915},
    ]
    cobs_cases = [
        {"name": name, "decoded": data.hex(), "encoded": cobs_encode(data).hex()}
        for name, data in _cobs_cases()
    ]
    messages = [
        {
            "name": name,
            "message": msg.NAME,
            "type": msg.TYPE,
            "seq": seq,
            "fields": message_to_dict(msg),
            "packet": encode_packet(msg, seq).hex(),
            "frame": encode_frame(msg, seq).hex(),
        }
        for name, msg, seq in _MESSAGES
    ]
    good = bytearray(encode_packet(VelocityCommand(0.5, 0.0, 0.0), 0))
    bad_crc = bytearray(good)
    bad_crc[-1] ^= 0x01
    bad_version = bytearray(good)
    bad_version[0] = 2
    unknown_type = bytearray(good)
    unknown_type[1] = 0x7F
    short_body = bytearray(good)
    short_body[3] = 11
    invalid = [
        ("corrupted crc", bytes(bad_crc), "crc"),
        ("version 2", _with_crc(bytes(bad_version[:-2])), "version"),
        ("unknown type", _with_crc(bytes(unknown_type[:-2])), "type"),
        ("length field mismatch", _with_crc(bytes(short_body[:-2])), "length"),
        ("too short", _with_crc(b"\x01\x05"), "short"),
    ]
    invalid_cases = [
        {
            "name": name,
            "packet": packet.hex(),
            "frame": (cobs_encode(packet) + b"\x00").hex(),
            "error": kind,
        }
        for name, packet, kind in invalid
    ]
    return {
        "description": (
            "Shared protocol vectors for encoder-odometry-firmware. Read by "
            "test/test_protocol_vectors (C++) and host/tests (Python). Regenerate with "
            "`odobridge vectors --write`; CI fails if the file is stale."
        ),
        "protocol_version": 1,
        "crc16": crc_cases,
        "cobs": cobs_cases,
        "messages": messages,
        "invalid": invalid_cases,
    }


def _with_crc(covered: bytes) -> bytes:
    return covered + crc16_ccitt(covered).to_bytes(2, "little")


def render(vectors: dict[str, Any]) -> str:
    return json.dumps(vectors, indent=2) + "\n"


def find_repo_file(start: Path | None = None) -> Path:
    """Locate the vectors file by walking up from ``start`` (default: cwd)."""
    here = (start or Path.cwd()).resolve()
    for directory in (here, *here.parents):
        candidate = directory / VECTORS_RELATIVE_PATH
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"{VECTORS_RELATIVE_PATH} not found above {here}")
