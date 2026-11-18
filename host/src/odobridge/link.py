"""Transports and a small client for talking to the firmware.

A transport moves bytes and owns the notion of time, so the same client code
drives a real serial port (wall-clock time) or the simulated firmware
(simulated time, advanced as the client waits).
"""

from __future__ import annotations

import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from typing import Any, Protocol

from .firmware_model import SimulatedFirmware
from .protocol import (
    PARAM_STATUS_NAMES,
    Diagnostics,
    FrameDecoder,
    Message,
    Odometry,
    Packet,
    ParamGet,
    ParamSet,
    ParamValue,
    SetPose,
    VelocityCommand,
    WheelState,
    encode_frame,
)


class Transport(Protocol):
    def write(self, data: bytes) -> None: ...
    def read_available(self) -> bytes: ...
    def now(self) -> float: ...
    def wait(self, seconds: float) -> None: ...
    def close(self) -> None: ...


class SerialTransport:
    """pyserial-backed transport. pyserial is imported only when used."""

    def __init__(self, port: str, baudrate: int = 921600, serial_obj: Any = None) -> None:
        if serial_obj is None:  # pragma: no cover - needs a real port
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError(
                    "pyserial is required for serial ports: pip install pyserial"
                ) from exc
            serial_obj = serial.Serial(port, baudrate=baudrate, timeout=0.01)
        self._serial = serial_obj

    def write(self, data: bytes) -> None:
        self._serial.write(data)

    def read_available(self) -> bytes:
        waiting = int(getattr(self._serial, "in_waiting", 0))
        data = self._serial.read(waiting if waiting > 0 else 1)
        return bytes(data)

    def now(self) -> float:
        return time.monotonic()

    def wait(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)

    def close(self) -> None:
        self._serial.close()


class SimTransport:
    """In-process transport to :class:`SimulatedFirmware`; time is simulated."""

    def __init__(self, firmware: SimulatedFirmware) -> None:
        self.firmware = firmware
        self._time = 0.0
        self._advanced = 0.0

    def write(self, data: bytes) -> None:
        self.firmware.write(data)

    def read_available(self) -> bytes:
        return self.firmware.read()

    def now(self) -> float:
        return self._time

    def wait(self, seconds: float) -> None:
        self._time += max(0.0, seconds)
        period = self.firmware.period_s
        while self._advanced + period <= self._time + 1e-12:
            self.firmware.step()
            self._advanced += period

    def close(self) -> None:
        pass


@dataclass
class Telemetry:
    """Latest messages received, plus counters."""

    wheel_state: WheelState | None = None
    odometry: Odometry | None = None
    diagnostics: Diagnostics | None = None
    received: int = 0
    seq_gaps: int = 0
    _last_seq: int | None = field(default=None, repr=False)

    def update(self, packet: Packet) -> None:
        self.received += 1
        if self._last_seq is not None and packet.seq != (self._last_seq + 1) & 0xFF:
            self.seq_gaps += 1
        self._last_seq = packet.seq
        message = packet.message
        if isinstance(message, WheelState):
            self.wheel_state = message
        elif isinstance(message, Odometry):
            self.odometry = message
        elif isinstance(message, Diagnostics):
            self.diagnostics = message


class Client:
    """Request/response helpers and the command-streaming loop."""

    def __init__(self, transport: Transport) -> None:
        self.transport = transport
        self.decoder = FrameDecoder()
        self.telemetry = Telemetry()
        self._seq = 0
        self._pending_params: list[ParamValue] = []

    def send(self, message: Message) -> None:
        self.transport.write(encode_frame(message, self._seq))
        self._seq = (self._seq + 1) & 0xFF

    def poll(self) -> Iterator[Packet]:
        """Read what is available and yield decoded packets."""
        for packet in self.decoder.feed(self.transport.read_available()):
            self.telemetry.update(packet)
            if isinstance(packet.message, ParamValue):
                self._pending_params.append(packet.message)
            yield packet

    def _wait_param(self, param_id: int, timeout: float) -> ParamValue:
        deadline = self.transport.now() + timeout
        while True:
            for _ in self.poll():
                pass
            for i, reply in enumerate(self._pending_params):
                if reply.id == param_id:
                    del self._pending_params[i]
                    return reply
            if self.transport.now() >= deadline:
                raise TimeoutError(f"no reply for parameter {param_id} within {timeout} s")
            self.transport.wait(0.01)

    def get_param(self, param_id: int, timeout: float = 1.0) -> ParamValue:
        self.send(ParamGet(param_id))
        return self._wait_param(param_id, timeout)

    def set_param(self, param_id: int, value: float, timeout: float = 1.0) -> ParamValue:
        self.send(ParamSet(param_id, value))
        return self._wait_param(param_id, timeout)

    def set_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self.send(SetPose(x, y, theta))

    def drive(
        self,
        command: VelocityCommand,
        duration: float,
        rate_hz: float = 20.0,
        on_packet: Callable[[Packet], None] | None = None,
        stop: bool = True,
    ) -> None:
        """Stream ``command`` at ``rate_hz`` for ``duration`` seconds, then a stop.

        The firmware's watchdog stops the motors if commands stop arriving, so
        commands are repeated rather than sent once.
        """
        period = 1.0 / rate_hz
        start = self.transport.now()
        next_send = start
        while self.transport.now() - start < duration:
            if self.transport.now() >= next_send:
                self.send(command)
                next_send += period
            for packet in self.poll():
                if on_packet is not None:
                    on_packet(packet)
            self.transport.wait(min(0.01, period))
        if stop:
            self.send(VelocityCommand(0.0, 0.0, 0.0))
            for packet in self.poll():
                if on_packet is not None:
                    on_packet(packet)


def describe_status(status: int) -> str:
    return PARAM_STATUS_NAMES.get(status, f"status {status}")
