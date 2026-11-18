"""The Python firmware model, checked against hand-derived values.

Most cases mirror a C++ Unity test with the same numbers, so the model and the
firmware core are held to the same expectations.
"""

from __future__ import annotations

import math

import pytest

from odobridge.firmware_model import (
    CommandWatchdog,
    DriveConfig,
    DriveController,
    EncoderSnapshot,
    MotorModel,
    PidConfig,
    Pose2D,
    SimulatedFirmware,
    VelocityConfig,
    VelocityEstimator,
    VelocityPid,
    compose,
    count_delta,
    elapsed_us,
    se2_exp,
    slew_limit,
    wrap_angle,
)
from odobridge.kinematics import Twist
from odobridge.protocol import (
    PARAM_OK,
    PARAM_OUT_OF_RANGE,
    PARAM_READ_ONLY,
    PARAM_UNKNOWN_ID,
    FrameDecoder,
    Odometry,
    ParamGet,
    ParamSet,
    ParamValue,
    SetPose,
    VelocityCommand,
    WheelState,
    encode_frame,
)


def constant_speed(cps: float, t_us: int, count0: int = 0, t0: int = 0) -> EncoderSnapshot:
    """Ideal encoder at constant speed: edge k at t0 + k / cps seconds."""
    n = math.floor(cps * t_us * 1e-6 + 1e-9)
    last_edge = t0 + round(n / cps * 1e6) if n > 0 else t0
    return EncoderSnapshot((count0 + n) & 0xFFFFFFFF, last_edge & 0xFFFFFFFF, 0, True)


def to_i32(v: int) -> int:
    return count_delta(v, 0)


def test_wrap_helpers() -> None:
    assert count_delta(-(2**31), 2**31 - 1) == 1
    assert count_delta(5, 10) == -5
    assert elapsed_us(10, 0xFFFFFFF0) == 26


def test_estimator_low_speed_period_is_exact_fixed_is_quantised() -> None:
    # 50 counts/s sampled at 100 Hz: the fixed-period estimate can only be
    # 0 or 100 counts/s, the period estimate is exactly 50.
    est = VelocityEstimator(VelocityConfig(200.0, 800.0, 250_000))
    fixed_seen = set()
    for k in range(0, 101):
        t = k * 10_000
        s = constant_speed(50.0, t)
        est.update(EncoderSnapshot(to_i32(s.count), s.last_edge_us), t)
        if k > 20:
            fixed_seen.add(round(est.fixed_cps))
            assert est.period_cps == pytest.approx(50.0)
            assert est.blended_cps == pytest.approx(50.0)
    assert fixed_seen == {0, 100}


def test_estimator_high_speed_uses_fixed_period() -> None:
    est = VelocityEstimator()
    for k in range(0, 20):
        t = k * 10_000
        est.update(constant_speed(100_000.0, t), t)
    assert est.fixed_cps == pytest.approx(100_000.0)
    assert est.weight == 1.0
    assert est.blended_cps == pytest.approx(100_000.0)


def test_estimator_count_and_clock_wraparound() -> None:
    # Start 1 ms before micros() wraps and 500 counts below INT32_MAX, run at
    # 20000 counts/s (200 counts per period): crossing both wraps must not
    # disturb the estimate.
    t0 = 0xFFFFFFFF - 1000
    c0 = 2**31 - 500
    est = VelocityEstimator()
    for k in range(0, 10):
        dt = k * 10_000
        s = constant_speed(20_000.0, dt, count0=c0, t0=t0)
        est.update(EncoderSnapshot(to_i32(s.count), s.last_edge_us), (t0 + dt) & 0xFFFFFFFF)
        if k >= 2:
            assert est.fixed_cps == pytest.approx(20_000.0)
            assert est.period_cps == pytest.approx(20_000.0, rel=1e-3)


def test_estimator_decays_to_zero_when_edges_stop() -> None:
    est = VelocityEstimator(VelocityConfig(200.0, 800.0, 250_000))
    for k in range(0, 30):
        t = k * 10_000
        est.update(constant_speed(50.0, t), t)
    last = constant_speed(50.0, 290_000)
    # Edges stop. 100 ms after the last edge the bound is 1e6/100e3 = 10 cps.
    est.update(last, last.last_edge_us + 100_000)
    assert est.period_cps == pytest.approx(10.0)
    est.update(last, last.last_edge_us + 300_000)
    assert est.period_cps == 0.0


def test_pid_feedforward_only_reaches_target_on_matched_plant() -> None:
    pid = VelocityPid(PidConfig(kp=0.0, ki=0.0, kff=0.03, ks=0.05, slew_rate=0.0))
    # u = 0.03 * 10 + 0.05 = 0.35
    assert pid.update(10.0, 0.0, 0.01) == pytest.approx(0.35)
    assert pid.update(-10.0, 0.0, 0.01) == pytest.approx(-0.35)
    assert pid.update(0.0, 0.0, 0.01) == 0.0


def test_pid_anti_windup_freezes_integrator_while_saturated() -> None:
    pid = VelocityPid(PidConfig(kp=0.1, ki=1.0, kff=0.0, slew_rate=0.0, output_limit=1.0))
    for _ in range(500):
        pid.update(100.0, 0.0, 0.01)  # hopeless target: output pinned at 1
    assert pid.output == 1.0
    # kp*e alone saturates, so the integrator never moved.
    assert pid.integral == 0.0
    # Once the target is reachable the output leaves saturation immediately.
    assert pid.update(5.0, 5.0, 0.01) == pytest.approx(0.0)


def test_pid_slew_limit() -> None:
    pid = VelocityPid(PidConfig(kp=1.0, ki=0.0, kff=0.0, slew_rate=2.0))
    assert pid.update(1.0, 0.0, 0.01) == pytest.approx(0.02)
    assert pid.update(1.0, 0.0, 0.01) == pytest.approx(0.04)
    assert slew_limit(0.0, 1.0, 0.0, 0.01) == 1.0


def test_watchdog_trips_once_and_needs_a_feed() -> None:
    wd = CommandWatchdog(timeout_us=100_000)
    assert not wd.update(0)
    assert wd.trips == 0
    wd.feed(1000)
    assert wd.update(101_000)
    assert not wd.update(101_001)
    assert not wd.update(500_000)
    assert wd.trips == 1
    wd.feed(600_000)
    assert wd.update(600_001)


def test_se2_exact_circle() -> None:
    # 100 steps of (dx = 2*pi/100, dtheta = 2*pi/100) is a unit circle: back to start.
    pose = Pose2D()
    step = 2.0 * math.pi / 100
    for _ in range(100):
        pose = compose(pose, se2_exp(step, 0.0, step))
    assert (pose.x, pose.y) == pytest.approx((0.0, 0.0), abs=1e-12)
    # Quarter circle of radius 1: (1, 1).
    q = se2_exp(math.pi / 2, 0.0, math.pi / 2)
    assert (q.x, q.y) == pytest.approx((1.0, 1.0))
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi) == pytest.approx(math.pi)


def test_controller_tracks_and_watchdog_stops() -> None:
    fw = SimulatedFirmware()
    fw.controller.command(Twist(0.3, 0.0, 0.0), fw.now_us)
    for _ in range(40):  # 0.4 s, within the 0.5 s watchdog
        fw.step()
    target = 0.3 / 0.05  # 6 rad/s
    for w in fw.controller.wheels:
        assert w.velocity == pytest.approx(target, rel=0.05)
    for _ in range(20):
        fw.step()
    assert fw.controller.watchdog.trips == 1
    assert all(w.duty == 0.0 for w in fw.controller.wheels)


def test_controller_rejects_non_finite_commands() -> None:
    c = DriveController()
    assert not c.command(Twist(float("nan"), 0.0, 0.0), 0)
    assert c.watchdog.expired


def test_simulated_firmware_speaks_the_protocol() -> None:
    fw = SimulatedFirmware(start_us=0xFFFFFFFF - 200_000)  # clock wraps mid-run
    fw.write(encode_frame(ParamSet(9, 5.0), 0))  # max_wheel_speed
    fw.write(encode_frame(ParamGet(21), 1))  # control_rate_hz, read-only
    fw.write(encode_frame(ParamSet(21, 50.0), 2))
    fw.write(encode_frame(ParamSet(14, 95.0), 3))
    fw.write(encode_frame(ParamGet(99), 4))
    decoder = FrameDecoder()
    replies = [p.message for p in decoder.feed(fw.read())]
    assert replies == [
        ParamValue(9, PARAM_OK, 5.0),
        ParamValue(21, PARAM_OK, 100.0),
        ParamValue(21, PARAM_READ_ONLY, 100.0),
        ParamValue(14, PARAM_OUT_OF_RANGE, 45.0),
        ParamValue(99, PARAM_UNKNOWN_ID, 0.0),
    ]
    # vx = 0.5 needs 10 rad/s per wheel; the 5 rad/s limit halves it.
    for _ in range(30):
        fw.write(encode_frame(VelocityCommand(0.5, 0.0, 0.0)))
        fw.step()
    assert fw.controller.scale == pytest.approx(0.5)
    messages = [p.message for p in decoder.feed(fw.read())]
    ws = [m for m in messages if isinstance(m, WheelState)][-1]
    assert ws.target == pytest.approx((5.0, 5.0, 5.0, 5.0))
    fw.write(encode_frame(SetPose(1.0, 2.0, 0.5)))
    odo = [p.message for p in decoder.feed(fw.read())]
    assert isinstance(odo[0], Odometry)
    assert (odo[0].x, odo[0].y, odo[0].theta) == pytest.approx((1.0, 2.0, 0.5))


def test_odometry_matches_ground_truth_on_an_arc() -> None:
    fw = SimulatedFirmware(DriveConfig())
    for _ in range(200):
        if fw.robot.now_us % 100_000 == 0:
            fw.controller.command(Twist(0.25, 0.1, 0.6), fw.now_us)
        fw.step()
    # step() samples the encoders before advancing the plant, so odometry is
    # one period behind; take one more sample to compare at the same instant.
    fw.controller.step(fw.robot.snapshots(), fw.now_us)
    truth, odo = fw.robot.true_pose, fw.controller.pose
    # Odometry only sees whole encoder counts (38000 per wheel rev), so it
    # lags the plant by less than one count per wheel: well under 1 mm.
    assert math.hypot(truth.x - odo.x, truth.y - odo.y) < 1e-3
    assert truth.theta == pytest.approx(odo.theta, abs=1e-4)
    assert truth.theta > 0.5


def test_motor_model_dead_band() -> None:
    m = MotorModel(duty_per_rad_s=0.05, deadband=0.1, tau=0.1)
    assert m.steady_state(0.05) == 0.0
    assert m.steady_state(0.6) == pytest.approx(10.0)
    assert m.steady_state(-0.6) == pytest.approx(-10.0)
