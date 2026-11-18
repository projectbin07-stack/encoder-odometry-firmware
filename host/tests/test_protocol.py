from __future__ import annotations

import struct
from typing import Any

import pytest

from odobridge.protocol import (
    MAX_FRAME_SIZE,
    FrameDecoder,
    Odometry,
    Packet,
    ProtocolError,
    VelocityCommand,
    WheelState,
    body_size,
    cobs_decode,
    cobs_encode,
    crc16_ccitt,
    decode_frame,
    decode_packet,
    duty_to_wire,
    encode_frame,
    encode_packet,
    message_from_dict,
    message_to_dict,
)
from odobridge.vectors import build_vectors, render

from .conftest import VECTORS_PATH


def test_crc_published_check_values(vectors: dict[str, Any]) -> None:
    # 0x29B1 is the catalogued check value of CRC-16/CCITT-FALSE for "123456789".
    assert crc16_ccitt(b"123456789") == 0x29B1
    for case in vectors["crc16"]:
        assert crc16_ccitt(case["ascii"].encode()) == case["crc"], case["name"]


def test_crc_is_chainable() -> None:
    assert crc16_ccitt(b"6789", crc16_ccitt(b"12345")) == 0x29B1


def test_cobs_vectors_both_directions(vectors: dict[str, Any]) -> None:
    for case in vectors["cobs"]:
        decoded = bytes.fromhex(case["decoded"])
        encoded = bytes.fromhex(case["encoded"])
        assert cobs_encode(decoded) == encoded, case["name"]
        assert cobs_decode(encoded) == decoded, case["name"]
        assert 0 not in encoded


def test_cobs_hand_examples() -> None:
    # Worked examples from Cheshire & Baker's COBS paper / common references.
    assert cobs_encode(b"\x00") == b"\x01\x01"
    assert cobs_encode(b"\x11\x22\x00\x33") == b"\x03\x11\x22\x02\x33"
    assert cobs_encode(bytes(range(1, 255))) == b"\xff" + bytes(range(1, 255))


@pytest.mark.parametrize(
    "bad",
    [b"\x00", b"\x05\x11\x22", b"\x03\x11\x00", b"\x02"],
)
def test_cobs_rejects_malformed(bad: bytes) -> None:
    # zero code byte / block past the end / zero inside a block / missing data byte
    with pytest.raises(ProtocolError) as info:
        cobs_decode(bad)
    assert info.value.kind == "cobs"


def test_message_vectors_encode_and_decode(vectors: dict[str, Any]) -> None:
    assert vectors["protocol_version"] == 1
    for case in vectors["messages"]:
        message = message_from_dict(case["message"], case["fields"])
        assert case["type"] == message.TYPE
        assert encode_packet(message, case["seq"]).hex() == case["packet"], case["name"]
        assert encode_frame(message, case["seq"]).hex() == case["frame"], case["name"]
        packet = decode_frame(bytes.fromhex(case["frame"]))
        assert packet.seq == case["seq"]
        assert message_to_dict(packet.message) == case["fields"], case["name"]


def test_invalid_vectors_rejected_with_the_right_reason(vectors: dict[str, Any]) -> None:
    for case in vectors["invalid"]:
        with pytest.raises(ProtocolError) as info:
            decode_packet(bytes.fromhex(case["packet"]))
        assert info.value.kind == case["error"], case["name"]


def test_vectors_file_is_up_to_date() -> None:
    assert VECTORS_PATH.read_text(encoding="utf-8") == render(build_vectors())


def test_velocity_command_layout_by_hand() -> None:
    # version 1, type 1, seq 0, length 12, then vx=0.5 (0x3F000000 LE), vy=0, wz=0.
    packet = encode_packet(VelocityCommand(0.5, 0.0, 0.0))
    assert packet[:4] == bytes([1, 1, 0, 12])
    assert packet[4:8] == bytes([0x00, 0x00, 0x00, 0x3F])
    assert packet[8:16] == bytes(8)
    assert len(packet) == 18
    assert struct.unpack("<H", packet[16:])[0] == crc16_ccitt(packet[:16])


def test_body_sizes() -> None:
    assert [body_size(t) for t in range(1, 9)] == [12, 60, 28, 41, 1, 5, 6, 12]
    assert body_size(0) == -1
    assert body_size(0x7F) == -1


def test_frame_decoder_handles_noise_split_and_errors() -> None:
    good = encode_frame(Odometry(1, 1.0, 2.0, 0.5, 0.0, 0.0, 0.0), seq=9)
    corrupted = bytearray(good)
    corrupted[5] ^= 0x40
    stream = b"\x00\x00" + good + bytes(corrupted) + b"\x07\x07\x00" + good
    decoder = FrameDecoder()
    out: list[Packet] = []
    for i in range(0, len(stream), 3):  # arbitrary split points
        out.extend(decoder.feed(stream[i : i + 3]))
    assert [p.seq for p in out] == [9, 9]
    assert decoder.frames_ok == 2
    assert decoder.crc_errors + decoder.frame_errors == 2


def test_frame_decoder_overflow_is_a_frame_error() -> None:
    decoder = FrameDecoder()
    assert list(decoder.feed(b"\x01" * (MAX_FRAME_SIZE + 10) + b"\x00")) == []
    assert decoder.frame_errors == 1
    good = encode_frame(VelocityCommand(1.0, 0.0, 0.0))
    assert len(list(decoder.feed(good))) == 1


def test_wheel_state_roundtrip_and_duty_fraction() -> None:
    ws = WheelState(
        5, (1, 2, 3, 4), (0.5, 1.5, 2.5, 3.5), (1.0, 2.0, 3.0, 4.0), (-10000, 0, 2500, 10000)
    )
    back = decode_frame(encode_frame(ws, 3)).message
    assert back == ws
    assert isinstance(back, WheelState)
    assert back.duty_fraction == (-1.0, 0.0, 0.25, 1.0)


@pytest.mark.parametrize(
    ("duty", "wire"),
    [
        (0.0, 0),
        (0.5, 5000),
        (-0.25, -2500),
        (1.5, 10000),
        (-2.0, -10000),
        (0.00005, 1),
        (-0.00005, -1),
        (float("nan"), 0),
        (float("inf"), 0),
    ],
)
def test_duty_to_wire(duty: float, wire: int) -> None:
    assert duty_to_wire(duty) == wire


def test_message_from_dict_unknown_name() -> None:
    with pytest.raises(KeyError):
        message_from_dict("nope", {})
