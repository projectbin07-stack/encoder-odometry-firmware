"""Mecanum and differential-drive kinematics, mirroring ``odocore/kinematics.cpp``.

Same conventions as the ``mecanum-kinematics`` library: REP-103 body frame,
wheel arrays in FL, FR, RL, RR order, positive wheel speed drives that side
forward, rollers forming an "X" seen from above.

Public API: :class:`Twist`, :class:`Geometry`, :func:`inverse`, :func:`forward`,
:func:`saturate` and the drive-type constants. See ``docs/host-api.md``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

__all__ = [
    "DIFFERENTIAL",
    "DRIVE_TYPES",
    "MECANUM",
    "Geometry",
    "Twist",
    "Wheels",
    "forward",
    "inverse",
    "saturate",
]

MECANUM = "mecanum"
DIFFERENTIAL = "differential"
DRIVE_TYPES = (MECANUM, DIFFERENTIAL)

Wheels = tuple[float, float, float, float]
"""Four wheel values in canonical FL, FR, RL, RR order."""


@dataclass(frozen=True)
class Twist:
    """Planar body twist in the REP-103 body frame (x forward, y left, z up).

    Attributes:
        vx: Forward velocity, m/s (positive = forward).
        vy: Lateral velocity, m/s (positive = to the robot's left).
        wz: Yaw rate, rad/s (positive = counter-clockwise seen from above).

    The same type also carries a body-frame displacement over one period
    (m, m, rad), which is what :func:`forward` returns for wheel angle
    increments. Immutable.
    """

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def scaled(self, factor: float) -> Twist:
        """Return a new twist with every component multiplied by ``factor``."""
        return Twist(self.vx * factor, self.vy * factor, self.wz * factor)


@dataclass(frozen=True)
class Geometry:
    """Drive-base geometry, the same fields as ``odocore::Geometry``.

    Attributes:
        wheel_radius: Wheel radius r, m.
        half_length: lx, robot centre to the wheel axles along x, m.
        half_width: ly, robot centre to the wheel centres along y, m.
        roller_angle_deg: Mecanum roller angle, degrees, strictly between 0 and
            90 (ignored by differential kinematics, but still validated).
        drive: :data:`MECANUM` or :data:`DIFFERENTIAL`.

    Raises:
        ValueError: on construction, if ``drive`` is not one of
            :data:`DRIVE_TYPES`, a length is not positive and finite, or the
            roller angle is outside (0, 90).

    Immutable; use :func:`dataclasses.replace` to derive a modified copy.
    """

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
        """t = tan(roller angle), dimensionless."""
        return math.tan(math.radians(self.roller_angle_deg))

    @property
    def rotation_arm(self) -> float:
        """k = ly + lx * tan(alpha), m: the yaw lever arm in the mecanum equations."""
        return self.half_width + self.half_length * self.tan_alpha


def inverse(geometry: Geometry, twist: Twist) -> Wheels:
    """Wheel angular speeds (rad/s) for a body twist, in FL, FR, RL, RR order.

    Mecanum, with t = :attr:`Geometry.tan_alpha` and k = :attr:`Geometry.rotation_arm`::

        r * w_FL = vx - t*vy - k*wz        r * w_FR = vx + t*vy + k*wz
        r * w_RL = vx + t*vy - k*wz        r * w_RR = vx - t*vy + k*wz

    Differential: left = (vx - wz*ly) / r for FL and RL, right =
    (vx + wz*ly) / r for FR and RR; ``vy`` is ignored. A positive wheel speed
    drives that wheel's side of the robot forward. No saturation is applied
    (see :func:`saturate`).
    """
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
    """Least-squares body twist (or displacement) from four wheel values.

    ``wheels`` are in FL, FR, RL, RR order: speeds in rad/s give a twist in
    m/s, m/s, rad/s; angle increments in rad give a body displacement in m, m,
    rad. Mecanum uses the closed form::

        vx = r/4    * ( w_FL + w_FR + w_RL + w_RR)
        vy = r/(4t) * (-w_FL + w_FR + w_RL - w_RR)
        wz = r/(4k) * (-w_FL + w_FR - w_RL + w_RR)

    Differential averages each side (left = FL, RL; right = FR, RR) and always
    returns ``vy = 0``.

    Raises:
        ValueError: if ``wheels`` does not hold exactly four values.
    """
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
    """Uniform, direction-preserving scaling so that no wheel exceeds ``max_speed``.

    Returns ``(scaled_wheels, scale)`` with ``scale`` in (0, 1]. All four
    wheels are multiplied by the same factor, so the commanded path is kept
    and only the speed along it drops. ``scale`` is 1 (wheels unchanged) when
    the peak ``|w|`` is already within ``max_speed`` or when ``max_speed`` is
    not positive or is infinite (saturation disabled).

    Raises:
        ValueError: if ``wheels`` does not hold exactly four values.
    """
    w = tuple(float(x) for x in wheels)
    if len(w) != 4:
        raise ValueError(f"expected 4 wheel values, got {len(w)}")
    peak = max(abs(x) for x in w)
    if not max_speed > 0.0 or math.isinf(max_speed) or peak <= max_speed:
        return (w[0], w[1], w[2], w[3]), 1.0
    scale = max_speed / peak
    return (w[0] * scale, w[1] * scale, w[2] * scale, w[3] * scale), scale
