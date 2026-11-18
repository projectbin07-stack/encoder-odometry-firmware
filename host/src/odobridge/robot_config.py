"""Robot configuration files shared with ``mecanum-kinematics``.

A config is a flat TOML file with the same keys ``mecanum-kinematics`` uses::

    wheel_radius = 0.050      # m
    half_length = 0.150       # m, lx
    half_width = 0.170        # m, ly
    roller_angle_deg = 45.0
    ticks_per_rev = 2000      # counts per encoder-shaft rev, already x4
    gear_ratio = 19.0
    max_wheel_speed = 25.0    # rad/s
    wheel_order = ["FL", "FR", "RL", "RR"]

so a file written for that library can be pushed to the firmware unchanged.
This module adds one optional key of its own, ``drive = "mecanum"`` or
``"differential"`` (default mecanum); ``mecanum-kinematics`` would reject that
key, so leave it out of files the two projects share.

The firmware always works in canonical FL, FR, RL, RR order. ``wheel_order``
only affects how :meth:`RobotConfig.to_config_order` presents wheel arrays.
"""

from __future__ import annotations

import math
import sys
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

from .kinematics import DRIVE_TYPES, MECANUM, Geometry

if sys.version_info >= (3, 11):
    import tomllib
else:  # pragma: no cover - exercised only on Python 3.10
    import tomli as tomllib

WHEEL_LABELS: tuple[str, str, str, str] = ("FL", "FR", "RL", "RR")


class ConfigError(ValueError):
    """Missing, unknown or invalid configuration values."""


@dataclass(frozen=True)
class RobotConfig:
    wheel_radius: float
    half_length: float
    half_width: float
    roller_angle_deg: float = 45.0
    ticks_per_rev: float = 1024.0
    gear_ratio: float = 1.0
    max_wheel_speed: float = math.inf
    wheel_order: tuple[str, ...] = WHEEL_LABELS
    name: str = "unnamed"
    description: str = ""
    drive: str = MECANUM

    def __post_init__(self) -> None:
        for key in ("wheel_radius", "half_length", "half_width", "ticks_per_rev", "gear_ratio"):
            value = getattr(self, key)
            if not isinstance(value, (int, float)) or not (value > 0 and math.isfinite(value)):
                raise ConfigError(f"{key} must be a positive finite number, got {value!r}")
        if not isinstance(self.max_wheel_speed, (int, float)) or not self.max_wheel_speed > 0:
            raise ConfigError(f"max_wheel_speed must be positive, got {self.max_wheel_speed!r}")
        if not 0.0 < self.roller_angle_deg < 90.0:
            raise ConfigError(
                f"roller_angle_deg must be strictly between 0 and 90, got {self.roller_angle_deg!r}"
            )
        order = tuple(str(label).upper() for label in self.wheel_order)
        if sorted(order) != sorted(WHEEL_LABELS):
            raise ConfigError(
                f"wheel_order must be a permutation of {list(WHEEL_LABELS)}, got {list(order)}"
            )
        object.__setattr__(self, "wheel_order", order)
        if self.drive not in DRIVE_TYPES:
            raise ConfigError(f"drive must be one of {list(DRIVE_TYPES)}, got {self.drive!r}")

    @property
    def ticks_per_wheel_rev(self) -> float:
        return float(self.ticks_per_rev) * float(self.gear_ratio)

    @property
    def geometry(self) -> Geometry:
        return Geometry(
            wheel_radius=float(self.wheel_radius),
            half_length=float(self.half_length),
            half_width=float(self.half_width),
            roller_angle_deg=float(self.roller_angle_deg),
            drive=self.drive,
        )

    def to_config_order(self, canonical: Sequence[float]) -> tuple[float, ...]:
        """Reorder an FL, FR, RL, RR array into this config's ``wheel_order``."""
        return tuple(canonical[WHEEL_LABELS.index(label)] for label in self.wheel_order)

    def firmware_params(self) -> list[tuple[str, float]]:
        """``(param name, value)`` pairs that make the firmware match this config."""
        out: list[tuple[str, float]] = [
            ("wheel_radius", float(self.wheel_radius)),
            ("half_length", float(self.half_length)),
            ("half_width", float(self.half_width)),
            ("roller_angle_deg", float(self.roller_angle_deg)),
            ("ticks_per_wheel_rev", self.ticks_per_wheel_rev),
            ("drive_type", 0.0 if self.drive == MECANUM else 1.0),
        ]
        if math.isfinite(self.max_wheel_speed):
            out.append(("max_wheel_speed", float(self.max_wheel_speed)))
        return out

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> RobotConfig:
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            raise ConfigError(f"unknown config keys: {unknown}; allowed keys: {sorted(known)}")
        missing = [k for k in ("wheel_radius", "half_length", "half_width") if k not in data]
        if missing:
            raise ConfigError(f"missing required config keys: {missing}")
        values = dict(data)
        if "wheel_order" in values:
            raw = values["wheel_order"]
            if isinstance(raw, str) or not isinstance(raw, (list, tuple)):
                raise ConfigError("wheel_order must be a list of four labels")
            values["wheel_order"] = tuple(raw)
        return cls(**values)

    @classmethod
    def from_toml(cls, path: str | Path) -> RobotConfig:
        file = Path(path)
        try:
            with file.open("rb") as handle:
                data = tomllib.load(handle)
        except tomllib.TOMLDecodeError as exc:
            raise ConfigError(f"{file}: invalid TOML: {exc}") from exc
        try:
            return cls.from_dict(data)
        except ConfigError as exc:
            raise ConfigError(f"{file}: {exc}") from exc
