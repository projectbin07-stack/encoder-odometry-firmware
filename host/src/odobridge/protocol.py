"""Encoder and decoder for the firmware's binary serial protocol (version 1).

Wire format (see ``docs/protocol.md``)::

    frame  = COBS(packet) 0x00
    packet = version:u8 type:u8 seq:u8 length:u8 body[length] crc16:u16le

All fields are little-endian, floats are IEEE-754 binary32 and the CRC is
CRC-16/CCITT-FALSE over ``version .. body``. This module is an independent
re-implementation of ``lib/odocore/src/odocore/protocol.cpp``; both are tested
against the byte vectors in ``test/vectors/protocol_vectors.json``.
"""

from __future__ import annotations

import struct
from collections.abc import Iterator
from dataclasses import dataclass, field, fields
from typing import ClassVar

PROTOCOL_VERSION = 1
HEADER_SIZE = 4
CRC_SIZE = 2
MAX_BODY_SIZE = 64
MAX_PACKET_SIZE = HEADER_SIZE + MAX_BODY_SIZE + CRC_SIZE
MAX_FRAME_SIZE = MAX_PACKET_SIZE + MAX_PACKET_SIZE // 254 + 1 + 1


class ProtocolError(ValueError):
    """A packet or frame could not be decoded."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


# --------------------------------------------------------------------- CRC
def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


# -------------------------------------------------------------------- COBS
def cobs_encode(data: bytes) -> bytes:
    """Consistent Overhead Byte Stuffing; the result contains no 0x00 byte."""
    out = bytearray([0])
    code_pos = 0
    code = 1
    for index, byte in enumerate(data):
        if byte != 0:
            out.append(byte)
            code += 1
        if byte == 0 or code == 0xFF:
            out[code_pos] = code
            code = 1
            code_pos = len(out)
            if byte == 0 or index + 1 < len(data):
                out.append(0)
    if code_pos < len(out):  # not when a full 254-byte block ended the input
        out[code_pos] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Inverse of :func:`cobs_encode`. Raises :class:`ProtocolError` on bad input."""
    out = bytearray()
    pos = 0
    while pos < len(data):
        code = data[pos]
        pos += 1
        if code == 0:
            raise ProtocolError("cobs", "zero byte inside a COBS block")
        end = pos + code - 1
        if end > len(data):
            raise ProtocolError("cobs", "COBS block runs past the end of the frame")
        chunk = data[pos:end]
        if 0 in chunk:
            raise ProtocolError("cobs", "zero byte inside a COBS block")
        out += chunk
        pos = end
        if code != 0xFF and pos < len(data):
            out.append(0)
    return bytes(out)


# ---------------------------------------------------------------- messages
@dataclass(frozen=True)
class VelocityCommand:
    """Host -> firmware: body twist in m/s, m/s, rad/s."""

    TYPE: ClassVar[int] = 0x01
    NAME: ClassVar[str] = "velocity_command"
    FORMAT: ClassVar[str] = "<3f"

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def to_values(self) -> tuple[float | int, ...]:
        return (self.vx, self.vy, self.wz)

    @classmethod
    def from_values(cls, v: tuple[float | int, ...]) -> VelocityCommand:
        return cls(float(v[0]), float(v[1]), float(v[2]))


@dataclass(frozen=True)
class WheelState:
    """Firmware -> host: per-wheel state in FL, FR, RL, RR order.

    ``duty`` is the raw wire value, duty cycle x 10000 (-10000..10000).
    """

    TYPE: ClassVar[int] = 0x02
    NAME: ClassVar[str] = "wheel_state"
    FORMAT: ClassVar[str] = "<I" + "iffh" * 4

    t_us: int = 0
    count: tuple[int, int, int, int] = (0, 0, 0, 0)
    velocity: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    target: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    duty: tuple[int, int, int, int] = (0, 0, 0, 0)

    @property
    def duty_fraction(self) -> tuple[float, ...]:
        return tuple(d / 10000.0 for d in self.duty)

    def to_values(self) -> tuple[float | int, ...]:
        values: list[float | int] = [self.t_us]
        for i in range(4):
            values += [self.count[i], self.velocity[i], self.target[i], self.duty[i]]
        return tuple(values)

    @classmethod
    def from_values(cls, v: tuple[float | int, ...]) -> WheelState:
        per = [v[1 + 4 * i : 5 + 4 * i] for i in range(4)]
        return cls(
            t_us=int(v[0]),
            count=(int(per[0][0]), int(per[1][0]), int(per[2][0]), int(per[3][0])),
            velocity=(float(per[0][1]), float(per[1][1]), float(per[2][1]), float(per[3][1])),
            target=(float(per[0][2]), float(per[1][2]), float(per[2][2]), float(per[3][2])),
            duty=(int(per[0][3]), int(per[1][3]), int(per[2][3]), int(per[3][3])),
        )


@dataclass(frozen=True)
class Odometry:
    """Firmware -> host: pose (m, m, rad wrapped to (-pi, pi]) and measured twist."""

    TYPE: ClassVar[int] = 0x03
    NAME: ClassVar[str] = "odometry"
    FORMAT: ClassVar[str] = "<I6f"

    t_us: int = 0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def to_values(self) -> tuple[float | int, ...]:
        return (self.t_us, self.x, self.y, self.theta, self.vx, self.vy, self.wz)

    @classmethod
    def from_values(cls, v: tuple[float | int, ...]) -> Odometry:
        return cls(int(v[0]), *(float(x) for x in v[1:7]))


#: Bits of :attr:`Diagnostics.flags`.
DIAG_WATCHDOG_EXPIRED = 0x01
DIAG_HARDWARE_COUNTERS = 0x02
DIAG_SATURATED = 0x04


@dataclass(frozen=True)
class Diagnostics:
    """Firmware -> host: error counters and status flags."""

    TYPE: ClassVar[int] = 0x04
    NAME: ClassVar[str] = "diagnostics"
    FORMAT: ClassVar[str] = "<I4I5IB"

    t_us: int = 0
    illegal_transitions: tuple[int, int, int, int] = (0, 0, 0, 0)
    loop_overruns: int = 0
    watchdog_trips: int = 0
    max_loop_us: int = 0
    rx_crc_errors: int = 0
    rx_frame_errors: int = 0
    flags: int = 0

    @property
    def watchdog_expired(self) -> bool:
        return bool(self.flags & DIAG_WATCHDOG_EXPIRED)

    @property
    def hardware_counters(self) -> bool:
        return bool(self.flags & DIAG_HARDWARE_COUNTERS)

    @property
    def saturated(self) -> bool:
        return bool(self.flags & DIAG_SATURATED)

    def to_values(self) -> tuple[float | int, ...]:
        return (
            self.t_us,
            *self.illegal_transitions,
            self.loop_overruns,
            self.watchdog_trips,
            self.max_loop_us,
            self.rx_crc_errors,
            self.rx_frame_errors,
            self.flags,
        )

    @classmethod
    def from_values(cls, v: tuple[float | int, ...]) -> Diagnostics:
        i = [int(x) for x in v]
        return cls(i[0], (i[1], i[2], i[3], i[4]), i[5], i[6], i[7], i[8], i[9], i[10])


@dataclass(frozen=True)
class ParamGet:
    """Host -> firmware: read one parameter."""

    TYPE: ClassVar[int] = 0x05
    NAME: ClassVar[str] = "param_get"
    FORMAT: ClassVar[str] = "<B"

    id: int = 0

    def to_values(self) -> tuple[float | int, ...]:
        return (self.id,)

    @classmethod
    def from_values(cls, v: tuple[float | int, ...]) -> ParamGet:
        return cls(int(v[0]))


@dataclass(frozen=True)
class ParamSet:
    """Host -> firmware: write one parameter."""

    TYPE: ClassVar[int] = 0x06
    NAME: ClassVar[str] = "param_set"
    FORMAT: ClassVar[str] = "<Bf"

    id: int = 0
    value: float = 0.0

    def to_values(self) -> tuple[float | int, ...]:
        return (self.id, self.value)

    @classmethod
    def from_values(cls, v: tuple[float | int, ...]) -> ParamSet:
        return cls(int(v[0]), float(v[1]))


#: Values of :attr:`ParamValue.status`.
PARAM_OK = 0
PARAM_UNKNOWN_ID = 1
PARAM_OUT_OF_RANGE = 2
PARAM_READ_ONLY = 3
PARAM_STATUS_NAMES = {
    PARAM_OK: "ok",
    PARAM_UNKNOWN_ID: "unknown id",
    PARAM_OUT_OF_RANGE: "out of range",
    PARAM_READ_ONLY: "read-only",
}


@dataclass(frozen=True)
class ParamValue:
    """Firmware -> host: reply to a get or set; ``value`` is the current value."""

    TYPE: ClassVar[int] = 0x07
    NAME: ClassVar[str] = "param_value"
    FORMAT: ClassVar[str] = "<BBf"

    id: int = 0
    status: int = PARAM_OK
    value: float = 0.0

    def to_values(self) -> tuple[float | int, ...]:
        return (self.id, self.status, self.value)

    @classmethod
    def from_values(cls, v: tuple[float | int, ...]) -> ParamValue:
        return cls(int(v[0]), int(v[1]), float(v[2]))


@dataclass(frozen=True)
class SetPose:
    """Host -> firmware: overwrite the odometry pose (all zero = reset)."""

    TYPE: ClassVar[int] = 0x08
    NAME: ClassVar[str] = "set_pose"
    FORMAT: ClassVar[str] = "<3f"

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def to_values(self) -> tuple[float | int, ...]:
        return (self.x, self.y, self.theta)

    @classmethod
    def from_values(cls, v: tuple[float | int, ...]) -> SetPose:
        return cls(float(v[0]), float(v[1]), float(v[2]))


Message = (
    VelocityCommand
    | WheelState
    | Odometry
    | Diagnostics
    | ParamGet
    | ParamSet
    | ParamValue
    | SetPose
)
MessageType = (
    type[VelocityCommand]
    | type[WheelState]
    | type[Odometry]
    | type[Diagnostics]
    | type[ParamGet]
    | type[ParamSet]
    | type[ParamValue]
    | type[SetPose]
)

MESSAGE_TYPES: dict[int, MessageType] = {
    cls.TYPE: cls
    for cls in (
        VelocityCommand,
        WheelState,
        Odometry,
        Diagnostics,
        ParamGet,
        ParamSet,
        ParamValue,
        SetPose,
    )
}


def body_size(type_id: int) -> int:
    """Body length in bytes for a message type, or -1 if unknown."""
    cls = MESSAGE_TYPES.get(type_id)
    return struct.calcsize(cls.FORMAT) if cls is not None else -1


@dataclass(frozen=True)
class Packet:
    """A decoded packet: the message plus its sequence number."""

    message: Message
    seq: int = 0


def encode_packet(message: Message, seq: int = 0) -> bytes:
    """Header + body + CRC (not yet COBS-encoded)."""
    body = struct.pack(message.FORMAT, *message.to_values())
    header = struct.pack("<BBBB", PROTOCOL_VERSION, message.TYPE, seq & 0xFF, len(body))
    covered = header + body
    return covered + struct.pack("<H", crc16_ccitt(covered))


def encode_frame(message: Message, seq: int = 0) -> bytes:
    """Complete wire frame including the trailing 0x00 delimiter."""
    return cobs_encode(encode_packet(message, seq)) + b"\x00"


def decode_packet(packet: bytes) -> Packet:
    """Parse a COBS-decoded packet. Raises :class:`ProtocolError` with ``kind``
    one of ``short``, ``crc``, ``version``, ``type``, ``length``."""
    if len(packet) < HEADER_SIZE + CRC_SIZE:
        raise ProtocolError("short", f"packet of {len(packet)} bytes is too short")
    covered, (crc,) = packet[:-CRC_SIZE], struct.unpack("<H", packet[-CRC_SIZE:])
    if crc16_ccitt(covered) != crc:
        raise ProtocolError("crc", "CRC mismatch")
    version, type_id, seq, length = covered[:HEADER_SIZE]
    if version != PROTOCOL_VERSION:
        raise ProtocolError("version", f"unsupported protocol version {version}")
    cls = MESSAGE_TYPES.get(type_id)
    if cls is None:
        raise ProtocolError("type", f"unknown message type 0x{type_id:02x}")
    expected = struct.calcsize(cls.FORMAT)
    body = covered[HEADER_SIZE:]
    if length != expected or len(body) != expected:
        raise ProtocolError(
            "length", f"{cls.NAME}: expected {expected} body bytes, got {length}/{len(body)}"
        )
    return Packet(cls.from_values(struct.unpack(cls.FORMAT, body)), seq)


def decode_frame(frame: bytes) -> Packet:
    """Decode one frame, with or without its trailing delimiter."""
    if frame.endswith(b"\x00"):
        frame = frame[:-1]
    return decode_packet(cobs_decode(frame))


@dataclass
class FrameDecoder:
    """Incremental decoder for a byte stream, mirroring the firmware's
    ``FrameReceiver`` (same buffer limit, same error classes)."""

    frames_ok: int = 0
    crc_errors: int = 0
    frame_errors: int = 0
    _buffer: bytearray = field(default_factory=bytearray)
    _overflow: bool = False

    def feed(self, data: bytes) -> Iterator[Packet]:
        """Consume bytes; yield every valid packet completed by them."""
        for byte in data:
            if byte != 0:
                if len(self._buffer) < MAX_FRAME_SIZE:
                    self._buffer.append(byte)
                else:
                    self._overflow = True
                continue
            chunk, overflow = bytes(self._buffer), self._overflow
            self._buffer.clear()
            self._overflow = False
            if not chunk:
                continue
            if overflow:
                self.frame_errors += 1
                continue
            try:
                packet = decode_packet(cobs_decode(chunk))
            except ProtocolError as exc:
                if exc.kind == "crc":
                    self.crc_errors += 1
                else:
                    self.frame_errors += 1
                continue
            self.frames_ok += 1
            yield packet


def duty_to_wire(duty: float) -> int:
    """Duty cycle in [-1, 1] -> int16 x 10000, rounded half away from zero."""
    if duty != duty or duty in (float("inf"), float("-inf")):
        return 0
    scaled = max(-10000.0, min(10000.0, duty * 10000.0))
    return int(scaled + 0.5) if scaled >= 0 else -int(-scaled + 0.5)


def message_to_dict(message: Message) -> dict[str, object]:
    """Field names and values, as used in the shared JSON vectors."""
    out: dict[str, object] = {}
    for f in fields(message):
        value = getattr(message, f.name)
        out[f.name] = list(value) if isinstance(value, tuple) else value
    return out


def message_from_dict(name: str, fields: dict[str, object]) -> Message:
    """Inverse of :func:`message_to_dict` for a message type ``NAME``."""
    for cls in MESSAGE_TYPES.values():
        if name == cls.NAME:
            kwargs = {k: tuple(v) if isinstance(v, list) else v for k, v in fields.items()}
            return cls(**kwargs)  # type: ignore[arg-type]
    raise KeyError(f"unknown message name {name!r}")
