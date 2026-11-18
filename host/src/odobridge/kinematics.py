"""Mecanum and differential-drive kinematics, mirroring ``odocore/kinematics.cpp``.

Same conventions as the ``mecanum-kinematics`` library: REP-103 body frame,
wheel arrays in FL, FR, RL, RR order, positive wheel speed drives that side
forward, rollers forming an "X" seen from above.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

MECANUM = "mecanum"
DIFFERENTIAL = "differential"
DRIVE_TYPES = (MECANUM, DIFFERENTIAL)

Wheels = tuple[float, float, float, float]


@dataclass(frozen=True)
class Twist:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def scaled(self, factor: float) -> Twist:
        return Twist(self.vx * factor, self.vy * factor, self.wz * factor)


@dataclass(frozen=True)
class Geometry:
    wheel_radius: float = 0.05
    half_length: float = 0.15
    half_width: float = 0.17
    roller_angle_deg: float = 45.0
    drive: str = MECANUM

    def __post_init__(self) -> None:
        if self.drive not in DRIVE_TYPES:
            raise ValueError(f"drive must be one of {DRIVE_TYPES}, got {self.drive!r}")
        for name in ("wheel_radius", "half_length", "half_width"):
            value = getattr(self, name)
            if not (value > 0.0 and math.isfinite(value)):
                raise ValueError(f"{name} must be positive and finite, got {value!r}")
        if not 0.0 < self.roller_angle_deg < 90.0:
            raise ValueError(f"roller_angle_deg must be in (0, 90), got {self.roller_angle_deg!r}")

    @property
    def tan_alpha(self) -> float:
        return math.tan(math.radians(self.roller_angle_deg))

    @property
    def rotation_arm(self) -> float:
        """k = ly + lx * tan(alpha)."""
        return self.half_width + self.half_length * self.tan_alpha


def inverse(geometry: Geometry, twist: Twist) -> Wheels:
    """Wheel speeds (rad/s) for a body twist, FL FR RL RR."""
    r = geometry.wheel_radius
    if geometry.drive == DIFFERENTIAL:
        left = (twist.vx - twist.wz * geometry.half_width) / r
        right = (twist.vx + twist.wz * geometry.half_width) / r
        return (left, right, left, right)
    t = geometry.tan_alpha
    k = geometry.rotation_arm
    return (
        (twist.vx - t * twist.vy - k * twist.wz) / r,
        (twist.vx + t * twist.vy + k * twist.wz) / r,
        (twist.vx + t * twist.vy - k * twist.wz) / r,
        (twist.vx - t * twist.vy + k * twist.wz) / r,
    )


def forward(geometry: Geometry, wheels: Sequence[float]) -> Twist:
    """Least-squares body twist (or displacement) from four wheel values."""
    if len(wheels) != 4:
        raise ValueError(f"expected 4 wheel values, got {len(wheels)}")
    fl, fr, rl, rr = (float(w) for w in wheels)
    r = geometry.wheel_radius
    if geometry.drive == DIFFERENTIAL:
        left = 0.5 * (fl + rl)
        right = 0.5 * (fr + rr)
        return Twist(
            0.5 * r * (left + right), 0.0, r * (right - left) / (2.0 * geometry.half_width)
        )
    t = geometry.tan_alpha
    k = geometry.rotation_arm
    return Twist(
        0.25 * r * (fl + fr + rl + rr),
        0.25 * r / t * (-fl + fr + rl - rr),
        0.25 * r / k * (-fl + fr - rl + rr),
    )


def saturate(wheels: Sequence[float], max_speed: float) -> tuple[Wheels, float]:
    """Uniform, direction-preserving scaling so that no wheel exceeds ``max_speed``."""
    w = tuple(float(x) for x in wheels)
    if len(w) != 4:
        raise ValueError(f"expected 4 wheel values, got {len(w)}")
    peak = max(abs(x) for x in w)
    if not max_speed > 0.0 or math.isinf(max_speed) or peak <= max_speed:
        return (w[0], w[1], w[2], w[3]), 1.0
    scale = max_speed / peak
    return (w[0] * scale, w[1] * scale, w[2] * scale, w[3] * scale), scale
