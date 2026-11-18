from __future__ import annotations

import pytest

from odobridge.firmware_model import SimulatedFirmware
from odobridge.link import Client, SerialTransport, SimTransport, Telemetry
from odobridge.protocol import Odometry, Packet, ParamGet, ParamValue, decode_frame, encode_frame


class FakeSerial:
    """Stands in for serial.Serial: replays canned bytes, records writes."""

    def __init__(self, incoming: bytes) -> None:
        self.incoming = bytearray(incoming)
        self.written = bytearray()
        self.closed = False

    @property
    def in_waiting(self) -> int:
        return len(self.incoming)

    def read(self, n: int) -> bytes:
        data = bytes(self.incoming[:n])
        del self.incoming[:n]
        return data

    def write(self, data: bytes) -> int:
        self.written += data
        return len(data)

    def close(self) -> None:
        self.closed = True


def test_client_over_fake_serial() -> None:
    reply = encode_frame(ParamValue(1, 0, 0.25), seq=0)
    fake = FakeSerial(reply)
    client = Client(SerialTransport("fake", serial_obj=fake))
    value = client.get_param(1, timeout=0.5)
    assert value == ParamValue(1, 0, 0.25)
    # The request on the wire: PARAM_GET id 1, seq 0.
    request = decode_frame(bytes(fake.written))
    assert request == Packet(ParamGet(1), 0)
    client.transport.close()
    assert fake.closed


def test_client_param_timeout() -> None:
    client = Client(SerialTransport("fake", serial_obj=FakeSerial(b"")))
    with pytest.raises(TimeoutError):
        client.get_param(1, timeout=0.05)


def test_telemetry_counts_sequence_gaps() -> None:
    t = Telemetry()
    for seq in (250, 251, 253, 254, 255, 0):
        t.update(Packet(Odometry(seq), seq))
    assert t.seq_gaps == 1
    assert t.received == 6
    assert t.odometry == Odometry(0)


def test_sim_transport_advances_whole_periods() -> None:
    fw = SimulatedFirmware()
    transport = SimTransport(fw)
    transport.wait(0.025)
    assert fw.now_us == 20_000  # two 10 ms periods
    transport.wait(0.005)
    assert fw.now_us == 30_000
    assert transport.read_available()  # telemetry was queued
