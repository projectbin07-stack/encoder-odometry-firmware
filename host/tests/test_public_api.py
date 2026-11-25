"""The documented public API (docs/host-api.md) imports and behaves as documented.

Values are derived by hand from the formulas in the docstrings, with the
default configuration: r = 0.05 m, lx = 0.15 m, ly = 0.17 m, 45 deg rollers
(t = 1, k = 0.32 m), 38000 counts per wheel revolution, kp = 0.02, ki = 0.2,
kff = 0.03, slew_rate = 4 duty/s, 500 ms watchdog.
"""

from __future__ import annotations

import contextlib
import hashlib
import importlib
import io
import math
import re
from pathlib import Path

import pytest

import odobridge
from odobridge.firmware_model import (
    DriveConfig,
    DriveController,
    EncoderSnapshot,
    MotorModel,
    SimulatedFirmware,
    SimulatedRobot,
    wrap_angle,
)
from odobridge.kinematics import DIFFERENTIAL, Geometry, Twist, inverse
from odobridge.protocol import SetPose, VelocityCommand, duty_to_wire, encode_frame

DOC = Path(__file__).resolve().parents[2] / "docs" / "host-api.md"
RAD_PER_TICK = 2.0 * math.pi / 38000.0
T0 = 1_000_000


# ------------------------------------------------------------------ imports
def test_documented_imports_resolve_to_the_module_objects() -> None:
    from odobridge import DriveConfig as RootConfig
    from odobridge import DriveController as RootController
    from odobridge import Geometry as RootGeometry
    from odobridge import SimulatedRobot as RootRobot
    from odobridge import Twist as RootTwist
    from odobridge import duty_to_wire as root_duty_to_wire
    from odobridge import wrap_angle as root_wrap_angle

    assert RootConfig is DriveConfig
    assert RootController is DriveController
    assert RootRobot is SimulatedRobot
    assert root_wrap_angle is wrap_angle
    assert RootGeometry is Geometry
    assert RootTwist is Twist
    assert root_duty_to_wire is duty_to_wire


@pytest.mark.parametrize(
    ("module", "required"),
    [
        (
            "odobridge.firmware_model",
            {"DriveConfig", "DriveController", "SimulatedRobot", "wrap_angle"},
        ),
        ("odobridge.kinematics", {"Geometry", "Twist"}),
        ("odobridge.protocol", {"duty_to_wire"}),
        (
            "odobridge",
            {
                "DriveConfig",
                "DriveController",
                "SimulatedRobot",
                "wrap_angle",
                "Geometry",
                "Twist",
                "duty_to_wire",
            },
        ),
    ],
)
def test_all_lists_existing_names_including_the_supported_ones(
    module: str, required: set[str]
) -> None:
    mod = importlib.import_module(module)
    names = mod.__all__
    assert len(names) == len(set(names))
    assert required <= set(names)
    for name in names:
        assert hasattr(mod, name), name
    namespace: dict[str, object] = {}
    exec(f"from {module} import *", namespace)
    assert set(names) <= set(namespace)


def test_package_is_marked_typed() -> None:
    assert (Path(odobridge.__file__).parent / "py.typed").is_file()


def test_supported_names_are_documented() -> None:
    for obj in (
        DriveConfig,
        DriveController,
        DriveController.command,
        DriveController.step,
        SimulatedRobot,
        SimulatedRobot.advance,
        SimulatedRobot.snapshots,
        wrap_angle,
        Geometry,
        Twist,
        duty_to_wire,
    ):
        assert obj.__doc__ is not None, obj
        assert len(obj.__doc__.strip()) > 40, obj


# ---------------------------------------------------------- simple functions
def test_wrap_angle_documented_values() -> None:
    assert wrap_angle(math.pi) == math.pi
    assert wrap_angle(-math.pi) == math.pi
    assert wrap_angle(1.5 * math.pi) == pytest.approx(-0.5 * math.pi)
    assert wrap_angle(0.0) == 0.0
    assert wrap_angle(-10.0) == pytest.approx(-10.0 + 4.0 * math.pi)
    assert math.isnan(wrap_angle(math.nan))
    with pytest.raises(ValueError, match="math domain error"):
        wrap_angle(math.inf)


@pytest.mark.parametrize(
    ("duty", "wire"),
    [
        (1.0, 10000),
        (-0.5, -5000),
        (0.00005, 1),
        (-0.00005, -1),
        (2.0, 10000),
        (-2.0, -10000),
        (0.04, 400),
        (math.nan, 0),
        (math.inf, 0),
        (-math.inf, 0),
    ],
)
def test_duty_to_wire_documented_values(duty: float, wire: int) -> None:
    assert duty_to_wire(duty) == wire


def test_wheel_order_and_signs() -> None:
    g = Geometry()
    # Forward: every wheel turns forward, vx / r.
    assert inverse(g, Twist(vx=0.5)) == pytest.approx((10.0, 10.0, 10.0, 10.0))
    # Left (+y): FL and RR backwards, FR and RL forwards, t * vy / r.
    assert inverse(g, Twist(vy=0.1)) == pytest.approx((-2.0, 2.0, 2.0, -2.0))
    # Counter-clockwise: left side backwards, right side forwards, k * wz / r.
    assert inverse(g, Twist(wz=1.0)) == pytest.approx((-6.4, 6.4, -6.4, 6.4))
    diff = Geometry(drive=DIFFERENTIAL, half_width=0.2)
    assert inverse(diff, Twist(vx=0.1, vy=5.0, wz=1.0)) == pytest.approx((-2.0, 6.0, -2.0, 6.0))


def test_geometry_and_twist_are_immutable_and_validated() -> None:
    g = Geometry()
    with pytest.raises(AttributeError):
        g.wheel_radius = 0.1  # type: ignore[misc]
    with pytest.raises(ValueError, match="drive"):
        Geometry(drive="omni")
    with pytest.raises(ValueError, match="wheel_radius"):
        Geometry(wheel_radius=0.0)
    with pytest.raises(ValueError, match="roller_angle_deg"):
        Geometry(roller_angle_deg=90.0)
    assert Twist(1.0, -2.0, 0.5).scaled(2.0) == Twist(2.0, -4.0, 1.0)


# ----------------------------------------------------------- DriveController
def snaps(counts: tuple[int, int, int, int], t_us: int) -> list[EncoderSnapshot]:
    """Four encoders whose last edge is at the sample time."""
    return [EncoderSnapshot(c & 0xFFFFFFFF, t_us & 0xFFFFFFFF) for c in counts]


def test_first_step_only_primes() -> None:
    c = DriveController()
    assert c.command(Twist(vx=0.25), T0)
    assert c.step(snaps((100, 100, 100, 100), T0), T0) == [0.0] * 4
    assert (c.pose.x, c.pose.y, c.pose.theta) == (0.0, 0.0, 0.0)
    assert [w.count for w in c.wheels] == [100] * 4


def test_first_controlled_steps_are_slew_limited_with_anti_windup() -> None:
    c = DriveController()
    c.step(snaps((0, 0, 0, 0), T0), T0)
    assert c.command(Twist(vx=0.25), T0)
    assert c.targets == pytest.approx([5.0] * 4)  # 0.25 / 0.05
    # Wheels still: e = 5, u = kff*5 + kp*5 + ki*5*0.01 = 0.26, but the slew
    # limit allows 4 duty/s * 10 ms = 0.04 per period.
    d1 = c.step(snaps((0, 0, 0, 0), T0 + 10_000), T0 + 10_000)
    assert d1 == pytest.approx([0.04] * 4)
    assert [duty_to_wire(d) for d in d1] == [400] * 4
    # The clipped output is pushing the same way as the error: the integrator
    # is held, so the next period is again 0.26 limited to 0.08.
    d2 = c.step(snaps((0, 0, 0, 0), T0 + 20_000), T0 + 20_000)
    assert d2 == pytest.approx([0.08] * 4)
    assert [w.target for w in c.wheels] == pytest.approx([5.0] * 4)
    assert [w.duty for w in c.wheels] == pytest.approx([0.08] * 4)


def test_odometry_integrates_even_without_commands() -> None:
    c = DriveController()
    c.step(snaps((0, 0, 0, 0), T0), T0)
    t1 = T0 + 10_000
    duty = c.step(snaps((1000, 1000, 1000, 1000), t1), t1)
    assert duty == [0.0] * 4  # watchdog never fed
    assert c.watchdog.expired
    assert c.watchdog.trips == 0  # starting expired is not a trip
    # 1000 counts in 10 ms on every wheel: straight ahead.
    assert c.pose.x == pytest.approx(0.05 * 1000 * RAD_PER_TICK)
    assert c.pose.y == 0.0
    assert c.pose.theta == 0.0
    assert c.measured.vx == pytest.approx(0.05 * 100_000 * RAD_PER_TICK)
    assert [w.velocity for w in c.wheels] == pytest.approx([100_000 * RAD_PER_TICK] * 4)


def test_pose_heading_is_not_wrapped() -> None:
    c = DriveController()
    c.step(snaps((0, 0, 0, 0), T0), T0)
    # Spin in place: left wheels back, right wheels forward. Per period
    # dtheta = r * n * rad_per_tick / k.
    n = 40_000
    dtheta = 0.05 * n * RAD_PER_TICK / 0.32
    for i in range(1, 5):
        t = T0 + i * 10_000
        c.step(snaps((-n * i, n * i, -n * i, n * i), t), t)
    assert c.pose.theta == pytest.approx(4 * dtheta)
    assert c.pose.theta > math.pi  # accumulated, not wrapped
    assert -math.pi < wrap_angle(c.pose.theta) <= math.pi
    assert wrap_angle(c.pose.theta) == pytest.approx(4 * dtheta - 2 * math.pi)
    assert c.pose.x == pytest.approx(0.0, abs=1e-12)
    assert c.pose.y == pytest.approx(0.0, abs=1e-12)


def test_watchdog_timeout_boundary_and_stop() -> None:
    c = DriveController()
    c.step(snaps((0, 0, 0, 0), T0), T0)
    c.command(Twist(vx=0.25), T0)
    t_ok = T0 + 500_000  # exactly the timeout: still fresh
    assert c.step(snaps((0, 0, 0, 0), t_ok), t_ok) != [0.0] * 4
    assert not c.watchdog.expired
    t_late = t_ok + 1
    assert c.step(snaps((0, 0, 0, 0), t_late), t_late) == [0.0] * 4
    assert c.watchdog.expired
    assert c.watchdog.trips == 1
    assert c.targets == [0.0] * 4
    assert c.active == Twist()
    assert c.scale == 1.0
    assert [w.duty for w in c.wheels] == [0.0] * 4


def test_watchdog_uses_the_wrapping_clock() -> None:
    c = DriveController()
    start = 0xFFFFFFFF - 5_000
    c.step(snaps((0, 0, 0, 0), start), start)
    c.command(Twist(vx=0.25), start)
    later = (start + 10_000) & 0xFFFFFFFF  # wrapped past zero
    assert later < start
    assert c.step(snaps((0, 0, 0, 0), later), later) == pytest.approx([0.04] * 4)
    assert not c.watchdog.expired


def test_command_saturates_uniformly_and_rejects_non_finite() -> None:
    c = DriveController()
    assert c.command(Twist(vx=2.0), T0)  # 40 rad/s per wheel, limit 25
    assert c.scale == pytest.approx(0.625)
    assert c.targets == pytest.approx([25.0] * 4)
    assert c.active.vx == pytest.approx(1.25)
    assert not c.command(Twist(vx=math.nan), T0 + 1)
    assert c.targets == pytest.approx([25.0] * 4)  # unchanged


def test_differential_command_drops_vy() -> None:
    c = DriveController(DriveConfig(geometry=Geometry(drive=DIFFERENTIAL, half_width=0.2)))
    assert c.command(Twist(vx=0.1, vy=0.3, wz=1.0), T0)
    assert c.targets == pytest.approx([-2.0, 6.0, -2.0, 6.0])
    assert c.active == Twist(0.1, 0.0, 1.0)


# ------------------------------------------------------------ SimulatedRobot
def test_robot_first_order_step_response() -> None:
    g = Geometry()
    robot = SimulatedRobot(g, 38000.0, substeps=1, start_us=0xFFFFFFFF - 10_000)
    # duty 0.4: steady state (0.4 - 0.06) / 0.034 = 10 rad/s; one step of
    # length tau reaches 1 - 1/e of it; the angle is the trapezoid.
    robot.advance([0.4, 0.4, 0.4, 0.4], 0.08)
    w = 10.0 * (1.0 - math.exp(-1.0))
    assert robot.speed == pytest.approx([w] * 4)
    assert robot.angle == pytest.approx([0.5 * w * 0.08] * 4)
    assert robot.count == [math.floor(0.5 * w * 0.08 / RAD_PER_TICK)] * 4
    assert robot.now_us == (0xFFFFFFFF - 10_000 + 80_000) & 0xFFFFFFFF
    assert robot.true_pose.x == pytest.approx(0.05 * 0.5 * w * 0.08)
    assert MotorModel().steady_state(0.4) == pytest.approx(10.0)
    assert MotorModel().steady_state(-0.05) == 0.0  # inside the dead band


def test_robot_signs_and_int32_snapshots() -> None:
    robot = SimulatedRobot(Geometry(), 38000.0)
    robot.advance([-0.5, 0.5, -0.5, 0.5], 0.5)
    assert robot.count[0] < 0 < robot.count[1]
    assert robot.true_pose.theta > 0.0  # right side forward: counter-clockwise
    robot.count[2] = 2**31  # past int32 range
    assert robot.snapshots()[2].count == -(2**31)


# --------------------------------------------------------- closed loop, docs
def test_closed_loop_tracks_the_command_and_is_repeatable() -> None:
    def run() -> tuple[DriveController, SimulatedRobot]:
        config = DriveConfig()
        c = DriveController(config)
        robot = SimulatedRobot(config.geometry, config.ticks_per_wheel_rev)
        for k in range(300):
            if k % 10 == 0:
                c.command(Twist(vx=0.3, wz=-0.4), robot.now_us)
            robot.advance(c.step(robot.snapshots(), robot.now_us), 0.01)
        c.step(robot.snapshots(), robot.now_us)  # sample the final plant state
        return c, robot

    c, robot = run()
    assert c.measured.vx == pytest.approx(0.3, abs=0.01)
    assert c.measured.wz == pytest.approx(-0.4, abs=0.01)
    # Ideal encoders, no slip: odometry matches the ground truth to within a
    # few counts (one count moves a wheel 0.05 * 2 pi / 38000 = 8.3 um).
    assert c.pose.x == pytest.approx(robot.true_pose.x, abs=2e-5)
    assert c.pose.y == pytest.approx(robot.true_pose.y, abs=2e-5)
    assert c.pose.theta == pytest.approx(robot.true_pose.theta, abs=2e-5)
    c2, robot2 = run()
    assert (c2.pose, robot2.count, robot2.last_edge_us) == (c.pose, robot.count, robot.last_edge_us)


# Digest of every telemetry byte from a fixed SimulatedFirmware run, recorded
# with odobridge 0.1.0. A change here means the model's output changed.
TELEMETRY_SHA256 = "372c6078652f47b6518f3dc2f39296d7009f416aa4c66a0f8f676505adcb49ba"


def test_fixed_run_telemetry_is_unchanged() -> None:
    fw = SimulatedFirmware(start_us=0xFFFFFFFF - 1_500_000)
    digest = hashlib.sha256()
    seq = 0
    for k in range(300):
        if k < 150 and k % 10 == 0:
            fw.write(encode_frame(VelocityCommand(0.4, 0.2, 0.9), seq))
            seq += 1
        if k == 90:
            fw.write(encode_frame(SetPose(1.0, -2.0, 3.0), seq))
            seq += 1
        fw.step()
        digest.update(fw.read())
    assert fw.frames_sent == 127
    assert digest.hexdigest() == TELEMETRY_SHA256


def test_docs_example_runs_and_prints_the_documented_output() -> None:
    text = DOC.read_text(encoding="utf-8")
    section = text.split("## Example", 1)[1]
    code = re.search(r"```python\n(.*?)```", section, re.S)
    expected = re.search(r"```text\n(.*?)```", section, re.S)
    assert code is not None
    assert expected is not None
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        exec(compile(code.group(1), str(DOC), "exec"), {})
    assert out.getvalue() == expected.group(1)
