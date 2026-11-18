"""Kinematics checked against values computed by hand.

Geometry used throughout: r = 0.05 m, lx = 0.15 m, ly = 0.17 m, 45 degree
rollers, so tan(alpha) = 1 and k = ly + lx = 0.32 m. The same numbers appear in
test/test_kinematics/test_main.cpp.
"""

from __future__ import annotations

import math

import pytest

from odobridge.kinematics import DIFFERENTIAL, Geometry, Twist, forward, inverse, saturate

GEOM = Geometry(0.05, 0.15, 0.17, 45.0)


def approx4(values: tuple[float, ...]) -> object:
    return pytest.approx(values, abs=1e-9)


def test_inverse_pure_translation_and_rotation() -> None:
    # vx = 0.5: every wheel 0.5 / 0.05 = 10 rad/s.
    assert inverse(GEOM, Twist(0.5, 0.0, 0.0)) == approx4((10.0, 10.0, 10.0, 10.0))
    # vy = 0.2: t*vy/r = 4 rad/s, signs (-, +, +, -).
    assert inverse(GEOM, Twist(0.0, 0.2, 0.0)) == approx4((-4.0, 4.0, 4.0, -4.0))
    # wz = 1: k*wz/r = 0.32/0.05 = 6.4 rad/s, signs (-, +, -, +).
    assert inverse(GEOM, Twist(0.0, 0.0, 1.0)) == approx4((-6.4, 6.4, -6.4, 6.4))


def test_inverse_combined_and_forward_roundtrip() -> None:
    # Sum of the three cases above: FL 10-4-6.4, FR 10+4+6.4, RL 10+4-6.4, RR 10-4+6.4.
    wheels = inverse(GEOM, Twist(0.5, 0.2, 1.0))
    assert wheels == approx4((-0.4, 20.4, 7.6, 12.4))
    back = forward(GEOM, wheels)
    assert (back.vx, back.vy, back.wz) == pytest.approx((0.5, 0.2, 1.0), abs=1e-12)


def test_forward_matches_mecanum_kinematics_jacobian() -> None:
    # Re-derive the mecanum-kinematics Jacobian row by row:
    # r*w_i = vx + s_i*t*vy + (-y_i + s_i*t*x_i)*wz with roller signs
    # (-1, 1, 1, -1) and positions (lx,ly), (lx,-ly), (-lx,ly), (-lx,-ly).
    g = Geometry(0.04, 0.12, 0.2, 30.0)
    t = math.tan(math.radians(30.0))
    signs = (-1.0, 1.0, 1.0, -1.0)
    pos = ((0.12, 0.2), (0.12, -0.2), (-0.12, 0.2), (-0.12, -0.2))
    twist = Twist(0.3, -0.1, 0.7)
    expected = tuple(
        (twist.vx + s * t * twist.vy + (-y + s * t * x) * twist.wz) / 0.04
        for s, (x, y) in zip(signs, pos, strict=True)
    )
    assert inverse(g, twist) == approx4(expected)
    # The rotation arm for 30 degree rollers: 0.2 + 0.12 * tan(30 deg).
    assert g.rotation_arm == pytest.approx(0.2 + 0.12 * 0.5773502691896257)


def test_forward_least_squares_with_inconsistent_wheels() -> None:
    # One wheel slipping: FL reads 2 rad/s more than a pure vx = 0.5 would give.
    twist = forward(GEOM, (12.0, 10.0, 10.0, 10.0))
    # vx = r/4 * 42 = 0.525; vy = r/4 * (-2) = -0.025; wz = r/(4k) * (-2) = -0.078125
    assert (twist.vx, twist.vy, twist.wz) == pytest.approx((0.525, -0.025, -0.078125))


def test_differential_drive() -> None:
    g = Geometry(0.05, 0.15, 0.17, 45.0, drive=DIFFERENTIAL)
    # left = (0.5 - 1*0.17)/0.05 = 6.6, right = (0.5 + 0.17)/0.05 = 13.4
    assert inverse(g, Twist(0.5, 0.3, 1.0)) == approx4((6.6, 13.4, 6.6, 13.4))
    back = forward(g, (6.6, 13.4, 6.6, 13.4))
    assert (back.vx, back.vy, back.wz) == pytest.approx((0.5, 0.0, 1.0))


def test_saturation_is_uniform() -> None:
    wheels, scale = saturate((30.0, -15.0, 0.0, 10.0), 20.0)
    assert scale == pytest.approx(2.0 / 3.0)
    assert wheels == approx4((20.0, -10.0, 0.0, 20.0 / 3.0))
    assert saturate((1.0, 2.0, 3.0, 4.0), math.inf)[1] == 1.0
    assert saturate((1.0, 2.0, 3.0, 4.0), 5.0)[1] == 1.0


@pytest.mark.parametrize(
    "kwargs",
    [{"wheel_radius": 0.0}, {"roller_angle_deg": 90.0}, {"drive": "omni"}, {"half_width": -1.0}],
)
def test_geometry_validation(kwargs: dict[str, object]) -> None:
    base: dict[str, object] = {"wheel_radius": 0.05, "half_length": 0.15, "half_width": 0.17}
    base.update(kwargs)
    with pytest.raises(ValueError):  # noqa: PT011
        Geometry(**base)  # type: ignore[arg-type]


def test_forward_needs_four_values() -> None:
    with pytest.raises(ValueError, match="4 wheel"):
        forward(GEOM, (1.0, 2.0, 3.0))
