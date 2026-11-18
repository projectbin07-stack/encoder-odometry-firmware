"""Firmware parameter table (mirror of ``lib/odocore/src/odocore/params.cpp``)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ParamInfo:
    id: int
    name: str
    minimum: float
    maximum: float
    read_only: bool = False
    integer: bool = False
    unit: str = ""
    help: str = ""


PARAMS: tuple[ParamInfo, ...] = (
    ParamInfo(1, "kp", 0.0, 10.0, unit="duty/(rad/s)", help="proportional gain"),
    ParamInfo(2, "ki", 0.0, 100.0, unit="duty/rad", help="integral gain"),
    ParamInfo(3, "kd", 0.0, 1.0, unit="duty/(rad/s^2)", help="derivative gain"),
    ParamInfo(4, "kff", 0.0, 1.0, unit="duty/(rad/s)", help="velocity feed-forward"),
    ParamInfo(5, "ks", 0.0, 1.0, unit="duty", help="static-friction feed-forward"),
    ParamInfo(6, "output_limit", 0.0, 1.0, unit="duty", help="duty ceiling"),
    ParamInfo(7, "slew_rate", 0.0, 1000.0, unit="duty/s", help="output rate limit, 0 = off"),
    ParamInfo(8, "d_cutoff_hz", 0.0, 1000.0, unit="Hz", help="derivative filter, 0 = off"),
    ParamInfo(9, "max_wheel_speed", 0.1, 1000.0, unit="rad/s", help="saturation limit"),
    ParamInfo(10, "watchdog_ms", 20.0, 10000.0, integer=True, unit="ms", help="command timeout"),
    ParamInfo(11, "wheel_radius", 0.001, 1.0, unit="m"),
    ParamInfo(12, "half_length", 0.001, 5.0, unit="m", help="lx"),
    ParamInfo(13, "half_width", 0.001, 5.0, unit="m", help="ly"),
    ParamInfo(14, "roller_angle_deg", 1.0, 89.0, unit="deg"),
    ParamInfo(15, "ticks_per_wheel_rev", 1.0, 10000000.0, unit="counts"),
    ParamInfo(16, "drive_type", 0.0, 1.0, integer=True, help="0 mecanum, 1 differential"),
    ParamInfo(17, "telemetry_divider", 1.0, 1000.0, integer=True, help="stream every N periods"),
    ParamInfo(18, "estimator_low_cps", 0.0, 1000000.0, unit="counts/s"),
    ParamInfo(19, "estimator_high_cps", 0.0, 1000000.0, unit="counts/s"),
    ParamInfo(20, "zero_timeout_ms", 1.0, 10000.0, integer=True, unit="ms"),
    ParamInfo(21, "control_rate_hz", 1.0, 10000.0, read_only=True, integer=True, unit="Hz"),
    ParamInfo(22, "protocol_version", 0.0, 255.0, read_only=True, integer=True),
)

BY_ID: dict[int, ParamInfo] = {p.id: p for p in PARAMS}
BY_NAME: dict[str, ParamInfo] = {p.name: p for p in PARAMS}


def lookup(name_or_id: str | int) -> ParamInfo:
    """Find a parameter by name or numeric id (as int or decimal string)."""
    if isinstance(name_or_id, int):
        key: str | int = name_or_id
    elif name_or_id.isdigit():
        key = int(name_or_id)
    else:
        key = name_or_id
    found = BY_ID.get(key) if isinstance(key, int) else BY_NAME.get(key)
    if found is None:
        raise KeyError(f"unknown parameter {name_or_id!r}; known: {', '.join(BY_NAME)}")
    return found
